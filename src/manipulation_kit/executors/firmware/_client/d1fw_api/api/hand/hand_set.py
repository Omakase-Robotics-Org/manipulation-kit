from http import HTTPStatus
from typing import Any
from urllib.parse import quote

import httpx

from ... import errors
from ...client import AuthenticatedClient, Client
from ...models.arm_side import ArmSide
from ...models.error_envelope import ErrorEnvelope
from ...models.hand_command import HandCommand
from ...models.hand_set_response_200 import HandSetResponse200
from ...types import Response


def _get_kwargs(
    side: ArmSide,
    *,
    body: HandCommand,
) -> dict[str, Any]:
    headers: dict[str, Any] = {}

    _kwargs: dict[str, Any] = {
        "method": "post",
        "url": "/v1/hand/{side}/set".format(
            side=quote(str(side), safe=""),
        ),
    }

    _kwargs["json"] = body.to_dict()

    headers["Content-Type"] = "application/json"

    _kwargs["headers"] = headers
    return _kwargs


def _parse_response(
    *, client: AuthenticatedClient | Client, response: httpx.Response
) -> ErrorEnvelope | HandSetResponse200 | None:
    if response.status_code == 200:
        response_200 = HandSetResponse200.from_dict(response.json())

        return response_200

    if response.status_code == 400:
        response_400 = ErrorEnvelope.from_dict(response.json())

        return response_400

    if response.status_code == 404:
        response_404 = ErrorEnvelope.from_dict(response.json())

        return response_404

    if response.status_code == 409:
        response_409 = ErrorEnvelope.from_dict(response.json())

        return response_409

    if response.status_code == 502:
        response_502 = ErrorEnvelope.from_dict(response.json())

        return response_502

    if response.status_code == 504:
        response_504 = ErrorEnvelope.from_dict(response.json())

        return response_504

    if client.raise_on_unexpected_status:
        raise errors.UnexpectedStatus(response.status_code, response.content)
    else:
        return None


def _build_response(
    *, client: AuthenticatedClient | Client, response: httpx.Response
) -> Response[ErrorEnvelope | HandSetResponse200]:
    return Response(
        status_code=HTTPStatus(response.status_code),
        content=response.content,
        headers=response.headers,
        parsed=_parse_response(client=client, response=response),
    )


def sync_detailed(
    side: ArmSide,
    *,
    client: AuthenticatedClient | Client,
    body: HandCommand,
) -> Response[ErrorEnvelope | HandSetResponse200]:
    """Command every axis of one dexterous hand

     `axes` is one value per axis in the model's own order, which `/v1/hand/{side}/capabilities`
    publishes as `axis_names`. `unit` is `frac` (0.0-1.0 of each axis's range, the canonical unit) or
    `wire` (the raw vendor unit: 0-255 on the LinkerHand O30, 0-10000 on the Leadshine DH116S). There is
    deliberately no degree unit: the O30's per-joint range of motion is unpublished and the direction of
    zero differs per joint type. A side that carries a gripper rather than a hand answers `unavailable`.

    Args:
        side (ArmSide): Selects one of the two physical arms.
        body (HandCommand): One commanded hand pose.

    Raises:
        errors.UnexpectedStatus: If the server returns an undocumented status code and Client.raise_on_unexpected_status is True.
        httpx.TimeoutException: If the request takes longer than Client.timeout.

    Returns:
        Response[ErrorEnvelope | HandSetResponse200]
    """

    kwargs = _get_kwargs(
        side=side,
        body=body,
    )

    response = client.get_httpx_client().request(
        **kwargs,
    )

    return _build_response(client=client, response=response)


def sync(
    side: ArmSide,
    *,
    client: AuthenticatedClient | Client,
    body: HandCommand,
) -> ErrorEnvelope | HandSetResponse200 | None:
    """Command every axis of one dexterous hand

     `axes` is one value per axis in the model's own order, which `/v1/hand/{side}/capabilities`
    publishes as `axis_names`. `unit` is `frac` (0.0-1.0 of each axis's range, the canonical unit) or
    `wire` (the raw vendor unit: 0-255 on the LinkerHand O30, 0-10000 on the Leadshine DH116S). There is
    deliberately no degree unit: the O30's per-joint range of motion is unpublished and the direction of
    zero differs per joint type. A side that carries a gripper rather than a hand answers `unavailable`.

    Args:
        side (ArmSide): Selects one of the two physical arms.
        body (HandCommand): One commanded hand pose.

    Raises:
        errors.UnexpectedStatus: If the server returns an undocumented status code and Client.raise_on_unexpected_status is True.
        httpx.TimeoutException: If the request takes longer than Client.timeout.

    Returns:
        ErrorEnvelope | HandSetResponse200
    """

    return sync_detailed(
        side=side,
        client=client,
        body=body,
    ).parsed


async def asyncio_detailed(
    side: ArmSide,
    *,
    client: AuthenticatedClient | Client,
    body: HandCommand,
) -> Response[ErrorEnvelope | HandSetResponse200]:
    """Command every axis of one dexterous hand

     `axes` is one value per axis in the model's own order, which `/v1/hand/{side}/capabilities`
    publishes as `axis_names`. `unit` is `frac` (0.0-1.0 of each axis's range, the canonical unit) or
    `wire` (the raw vendor unit: 0-255 on the LinkerHand O30, 0-10000 on the Leadshine DH116S). There is
    deliberately no degree unit: the O30's per-joint range of motion is unpublished and the direction of
    zero differs per joint type. A side that carries a gripper rather than a hand answers `unavailable`.

    Args:
        side (ArmSide): Selects one of the two physical arms.
        body (HandCommand): One commanded hand pose.

    Raises:
        errors.UnexpectedStatus: If the server returns an undocumented status code and Client.raise_on_unexpected_status is True.
        httpx.TimeoutException: If the request takes longer than Client.timeout.

    Returns:
        Response[ErrorEnvelope | HandSetResponse200]
    """

    kwargs = _get_kwargs(
        side=side,
        body=body,
    )

    response = await client.get_async_httpx_client().request(**kwargs)

    return _build_response(client=client, response=response)


async def asyncio(
    side: ArmSide,
    *,
    client: AuthenticatedClient | Client,
    body: HandCommand,
) -> ErrorEnvelope | HandSetResponse200 | None:
    """Command every axis of one dexterous hand

     `axes` is one value per axis in the model's own order, which `/v1/hand/{side}/capabilities`
    publishes as `axis_names`. `unit` is `frac` (0.0-1.0 of each axis's range, the canonical unit) or
    `wire` (the raw vendor unit: 0-255 on the LinkerHand O30, 0-10000 on the Leadshine DH116S). There is
    deliberately no degree unit: the O30's per-joint range of motion is unpublished and the direction of
    zero differs per joint type. A side that carries a gripper rather than a hand answers `unavailable`.

    Args:
        side (ArmSide): Selects one of the two physical arms.
        body (HandCommand): One commanded hand pose.

    Raises:
        errors.UnexpectedStatus: If the server returns an undocumented status code and Client.raise_on_unexpected_status is True.
        httpx.TimeoutException: If the request takes longer than Client.timeout.

    Returns:
        ErrorEnvelope | HandSetResponse200
    """

    return (
        await asyncio_detailed(
            side=side,
            client=client,
            body=body,
        )
    ).parsed
