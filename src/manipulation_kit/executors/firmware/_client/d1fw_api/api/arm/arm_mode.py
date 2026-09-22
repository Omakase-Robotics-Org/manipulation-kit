from http import HTTPStatus
from typing import Any
from urllib.parse import quote

import httpx

from ... import errors
from ...client import AuthenticatedClient, Client
from ...models.arm_mode_command import ArmModeCommand
from ...models.arm_mode_response_200 import ArmModeResponse200
from ...models.arm_side import ArmSide
from ...models.error_envelope import ErrorEnvelope
from ...types import Response


def _get_kwargs(
    side: ArmSide,
    *,
    body: ArmModeCommand,
) -> dict[str, Any]:
    headers: dict[str, Any] = {}

    _kwargs: dict[str, Any] = {
        "method": "post",
        "url": "/v1/arm/{side}/mode".format(
            side=quote(str(side), safe=""),
        ),
    }

    _kwargs["json"] = body.to_dict()

    headers["Content-Type"] = "application/json"

    _kwargs["headers"] = headers
    return _kwargs


def _parse_response(
    *, client: AuthenticatedClient | Client, response: httpx.Response
) -> ArmModeResponse200 | ErrorEnvelope | None:
    if response.status_code == 200:
        response_200 = ArmModeResponse200.from_dict(response.json())

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
) -> Response[ArmModeResponse200 | ErrorEnvelope]:
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
    body: ArmModeCommand,
) -> Response[ArmModeResponse200 | ErrorEnvelope]:
    """Set one arm's control mode

     The mode is selected by `mode` and carries that mode's own fields. `vel_ratio` and `acc_ratio` are
    fractions of full speed in `0.0..=1.0`, not percentages, and a value outside that range is rejected.

    One divergence remains between this route and the `arm.{side}.mode` WebSocket method, and it is in
    what an omitted field means: over REST `position` requires both ratios explicitly, while the
    WebSocket method defaults an omitted ratio to `0.1`. Every mode, `torque` included, and every field
    is otherwise reachable identically from both. Send the ratios explicitly and the two behave the
    same.

    Args:
        side (ArmSide): Selects one of the two physical arms.
        body (ArmModeCommand): The advertised request body of `POST /v1/arm/{side}/mode`.

            This is a *flattened, advertise-only* view of the tagged
            [`d1fw_core::ArmModeRequest`] enum with the arm lease's optional `holder`
            folded in as a top-level field. The daemon still deserializes the wire
            body into `ArmModeRequest` and reads `holder` off it separately (see
            `d1fw_front_rest::lease`); nothing deserializes into this struct. It exists
            only so the published document describes the body as one flat object.

            `openapi-python-client` silently skips any endpoint whose request body is
            an `allOf` containing a `oneOf`, which is exactly what the tagged enum plus
            a composed-in `holder` used to emit — so `POST /v1/arm/{side}/mode`, the
            most-used arm verb, was missing from every generated Python client. A
            single flat object with a `mode` string discriminator and every per-variant
            parameter optional generates cleanly. Which fields are required for a given
            `mode`, and their defaults, are documented on [`d1fw_core::ArmModeRequest`]
            and enforced by the daemon, not by this looser schema.

    Raises:
        errors.UnexpectedStatus: If the server returns an undocumented status code and Client.raise_on_unexpected_status is True.
        httpx.TimeoutException: If the request takes longer than Client.timeout.

    Returns:
        Response[ArmModeResponse200 | ErrorEnvelope]
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
    body: ArmModeCommand,
) -> ArmModeResponse200 | ErrorEnvelope | None:
    """Set one arm's control mode

     The mode is selected by `mode` and carries that mode's own fields. `vel_ratio` and `acc_ratio` are
    fractions of full speed in `0.0..=1.0`, not percentages, and a value outside that range is rejected.

    One divergence remains between this route and the `arm.{side}.mode` WebSocket method, and it is in
    what an omitted field means: over REST `position` requires both ratios explicitly, while the
    WebSocket method defaults an omitted ratio to `0.1`. Every mode, `torque` included, and every field
    is otherwise reachable identically from both. Send the ratios explicitly and the two behave the
    same.

    Args:
        side (ArmSide): Selects one of the two physical arms.
        body (ArmModeCommand): The advertised request body of `POST /v1/arm/{side}/mode`.

            This is a *flattened, advertise-only* view of the tagged
            [`d1fw_core::ArmModeRequest`] enum with the arm lease's optional `holder`
            folded in as a top-level field. The daemon still deserializes the wire
            body into `ArmModeRequest` and reads `holder` off it separately (see
            `d1fw_front_rest::lease`); nothing deserializes into this struct. It exists
            only so the published document describes the body as one flat object.

            `openapi-python-client` silently skips any endpoint whose request body is
            an `allOf` containing a `oneOf`, which is exactly what the tagged enum plus
            a composed-in `holder` used to emit — so `POST /v1/arm/{side}/mode`, the
            most-used arm verb, was missing from every generated Python client. A
            single flat object with a `mode` string discriminator and every per-variant
            parameter optional generates cleanly. Which fields are required for a given
            `mode`, and their defaults, are documented on [`d1fw_core::ArmModeRequest`]
            and enforced by the daemon, not by this looser schema.

    Raises:
        errors.UnexpectedStatus: If the server returns an undocumented status code and Client.raise_on_unexpected_status is True.
        httpx.TimeoutException: If the request takes longer than Client.timeout.

    Returns:
        ArmModeResponse200 | ErrorEnvelope
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
    body: ArmModeCommand,
) -> Response[ArmModeResponse200 | ErrorEnvelope]:
    """Set one arm's control mode

     The mode is selected by `mode` and carries that mode's own fields. `vel_ratio` and `acc_ratio` are
    fractions of full speed in `0.0..=1.0`, not percentages, and a value outside that range is rejected.

    One divergence remains between this route and the `arm.{side}.mode` WebSocket method, and it is in
    what an omitted field means: over REST `position` requires both ratios explicitly, while the
    WebSocket method defaults an omitted ratio to `0.1`. Every mode, `torque` included, and every field
    is otherwise reachable identically from both. Send the ratios explicitly and the two behave the
    same.

    Args:
        side (ArmSide): Selects one of the two physical arms.
        body (ArmModeCommand): The advertised request body of `POST /v1/arm/{side}/mode`.

            This is a *flattened, advertise-only* view of the tagged
            [`d1fw_core::ArmModeRequest`] enum with the arm lease's optional `holder`
            folded in as a top-level field. The daemon still deserializes the wire
            body into `ArmModeRequest` and reads `holder` off it separately (see
            `d1fw_front_rest::lease`); nothing deserializes into this struct. It exists
            only so the published document describes the body as one flat object.

            `openapi-python-client` silently skips any endpoint whose request body is
            an `allOf` containing a `oneOf`, which is exactly what the tagged enum plus
            a composed-in `holder` used to emit — so `POST /v1/arm/{side}/mode`, the
            most-used arm verb, was missing from every generated Python client. A
            single flat object with a `mode` string discriminator and every per-variant
            parameter optional generates cleanly. Which fields are required for a given
            `mode`, and their defaults, are documented on [`d1fw_core::ArmModeRequest`]
            and enforced by the daemon, not by this looser schema.

    Raises:
        errors.UnexpectedStatus: If the server returns an undocumented status code and Client.raise_on_unexpected_status is True.
        httpx.TimeoutException: If the request takes longer than Client.timeout.

    Returns:
        Response[ArmModeResponse200 | ErrorEnvelope]
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
    body: ArmModeCommand,
) -> ArmModeResponse200 | ErrorEnvelope | None:
    """Set one arm's control mode

     The mode is selected by `mode` and carries that mode's own fields. `vel_ratio` and `acc_ratio` are
    fractions of full speed in `0.0..=1.0`, not percentages, and a value outside that range is rejected.

    One divergence remains between this route and the `arm.{side}.mode` WebSocket method, and it is in
    what an omitted field means: over REST `position` requires both ratios explicitly, while the
    WebSocket method defaults an omitted ratio to `0.1`. Every mode, `torque` included, and every field
    is otherwise reachable identically from both. Send the ratios explicitly and the two behave the
    same.

    Args:
        side (ArmSide): Selects one of the two physical arms.
        body (ArmModeCommand): The advertised request body of `POST /v1/arm/{side}/mode`.

            This is a *flattened, advertise-only* view of the tagged
            [`d1fw_core::ArmModeRequest`] enum with the arm lease's optional `holder`
            folded in as a top-level field. The daemon still deserializes the wire
            body into `ArmModeRequest` and reads `holder` off it separately (see
            `d1fw_front_rest::lease`); nothing deserializes into this struct. It exists
            only so the published document describes the body as one flat object.

            `openapi-python-client` silently skips any endpoint whose request body is
            an `allOf` containing a `oneOf`, which is exactly what the tagged enum plus
            a composed-in `holder` used to emit — so `POST /v1/arm/{side}/mode`, the
            most-used arm verb, was missing from every generated Python client. A
            single flat object with a `mode` string discriminator and every per-variant
            parameter optional generates cleanly. Which fields are required for a given
            `mode`, and their defaults, are documented on [`d1fw_core::ArmModeRequest`]
            and enforced by the daemon, not by this looser schema.

    Raises:
        errors.UnexpectedStatus: If the server returns an undocumented status code and Client.raise_on_unexpected_status is True.
        httpx.TimeoutException: If the request takes longer than Client.timeout.

    Returns:
        ArmModeResponse200 | ErrorEnvelope
    """

    return (
        await asyncio_detailed(
            side=side,
            client=client,
            body=body,
        )
    ).parsed
