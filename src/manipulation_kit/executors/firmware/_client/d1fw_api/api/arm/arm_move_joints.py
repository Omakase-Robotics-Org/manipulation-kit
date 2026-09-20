from http import HTTPStatus
from typing import Any
from urllib.parse import quote

import httpx

from ... import errors
from ...client import AuthenticatedClient, Client
from ...models.arm_move_joints_response_200 import ArmMoveJointsResponse200
from ...models.arm_side import ArmSide
from ...models.error_envelope import ErrorEnvelope
from ...models.joints_request import JointsRequest
from ...types import Response


def _get_kwargs(
    side: ArmSide,
    *,
    body: JointsRequest,
) -> dict[str, Any]:
    headers: dict[str, Any] = {}

    _kwargs: dict[str, Any] = {
        "method": "post",
        "url": "/v1/arm/{side}/move_joints".format(
            side=quote(str(side), safe=""),
        ),
    }

    _kwargs["json"] = body.to_dict()

    headers["Content-Type"] = "application/json"

    _kwargs["headers"] = headers
    return _kwargs


def _parse_response(
    *, client: AuthenticatedClient | Client, response: httpx.Response
) -> ArmMoveJointsResponse200 | ErrorEnvelope | None:
    if response.status_code == 200:
        response_200 = ArmMoveJointsResponse200.from_dict(response.json())

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
) -> Response[ArmMoveJointsResponse200 | ErrorEnvelope]:
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
    body: JointsRequest,
) -> Response[ArmMoveJointsResponse200 | ErrorEnvelope]:
    """Command one arm to a joint target

     Angles are degrees. With `wait: true` the call blocks until the arm reports arrival.

    Args:
        side (ArmSide): Selects one of the two physical arms.
        body (JointsRequest): `POST /v1/arm/{side}/move_joints`.

    Raises:
        errors.UnexpectedStatus: If the server returns an undocumented status code and Client.raise_on_unexpected_status is True.
        httpx.TimeoutException: If the request takes longer than Client.timeout.

    Returns:
        Response[ArmMoveJointsResponse200 | ErrorEnvelope]
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
    body: JointsRequest,
) -> ArmMoveJointsResponse200 | ErrorEnvelope | None:
    """Command one arm to a joint target

     Angles are degrees. With `wait: true` the call blocks until the arm reports arrival.

    Args:
        side (ArmSide): Selects one of the two physical arms.
        body (JointsRequest): `POST /v1/arm/{side}/move_joints`.

    Raises:
        errors.UnexpectedStatus: If the server returns an undocumented status code and Client.raise_on_unexpected_status is True.
        httpx.TimeoutException: If the request takes longer than Client.timeout.

    Returns:
        ArmMoveJointsResponse200 | ErrorEnvelope
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
    body: JointsRequest,
) -> Response[ArmMoveJointsResponse200 | ErrorEnvelope]:
    """Command one arm to a joint target

     Angles are degrees. With `wait: true` the call blocks until the arm reports arrival.

    Args:
        side (ArmSide): Selects one of the two physical arms.
        body (JointsRequest): `POST /v1/arm/{side}/move_joints`.

    Raises:
        errors.UnexpectedStatus: If the server returns an undocumented status code and Client.raise_on_unexpected_status is True.
        httpx.TimeoutException: If the request takes longer than Client.timeout.

    Returns:
        Response[ArmMoveJointsResponse200 | ErrorEnvelope]
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
    body: JointsRequest,
) -> ArmMoveJointsResponse200 | ErrorEnvelope | None:
    """Command one arm to a joint target

     Angles are degrees. With `wait: true` the call blocks until the arm reports arrival.

    Args:
        side (ArmSide): Selects one of the two physical arms.
        body (JointsRequest): `POST /v1/arm/{side}/move_joints`.

    Raises:
        errors.UnexpectedStatus: If the server returns an undocumented status code and Client.raise_on_unexpected_status is True.
        httpx.TimeoutException: If the request takes longer than Client.timeout.

    Returns:
        ArmMoveJointsResponse200 | ErrorEnvelope
    """

    return (
        await asyncio_detailed(
            side=side,
            client=client,
            body=body,
        )
    ).parsed
