from http import HTTPStatus
from typing import Any
from urllib.parse import quote

import httpx

from ... import errors
from ...client import AuthenticatedClient, Client
from ...models.arm_brake_release_body import ArmBrakeReleaseBody
from ...models.arm_brake_release_response_200 import ArmBrakeReleaseResponse200
from ...models.arm_side import ArmSide
from ...models.error_envelope import ErrorEnvelope
from ...types import Response


def _get_kwargs(
    side: ArmSide,
    *,
    body: ArmBrakeReleaseBody,
) -> dict[str, Any]:
    headers: dict[str, Any] = {}

    _kwargs: dict[str, Any] = {
        "method": "post",
        "url": "/v1/arm/{side}/brake_release".format(
            side=quote(str(side), safe=""),
        ),
    }

    _kwargs["json"] = body.to_dict()

    headers["Content-Type"] = "application/json"

    _kwargs["headers"] = headers
    return _kwargs


def _parse_response(
    *, client: AuthenticatedClient | Client, response: httpx.Response
) -> ArmBrakeReleaseResponse200 | ErrorEnvelope | None:
    if response.status_code == 200:
        response_200 = ArmBrakeReleaseResponse200.from_dict(response.json())

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
) -> Response[ArmBrakeReleaseResponse200 | ErrorEnvelope]:
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
    body: ArmBrakeReleaseBody,
) -> Response[ArmBrakeReleaseResponse200 | ErrorEnvelope]:
    """Release one arm's holding brakes for hand guiding (the arm DROPS)

     DANGEROUS. Forces this arm's holding brakes OPEN (the controller's `BRAK0`/`BRAK1` = 2 override, the
    vendor's recovery recipe for an arm twisted outside its limits) so a person can move it by hand. The
    servos are off, so **nothing holds the arm: it falls under gravity the instant the brakes open
    unless somebody is supporting it.**

    The body must carry `{"confirm": "RELEASE_BRAKE"}` (otherwise `400`). Accepted only while the arm's
    servos are off — mode `idle` or `error`; any other mode is a `409` naming the call that idles it.
    Gated by the arm soft-kill latch and, while somebody holds it, by the arm lease (`x-arm-lease`).
    REST-only.

    The release is TIMED: `seconds` (`1..=120`, default `30`) after it lands the daemon engages the
    brakes itself. Sending it again while released restarts the window. The brakes are also engaged by
    `/v1/arm/{side}/brake_engage`, by `/v1/arm/{side}/estop`, and by the soft kill (which the daemon
    also runs on shutdown). While released, `mode`, `recover` and a trajectory are refused on this arm,
    `ArmState` carries `brakes_released: true` and preflight lists it as blocking.

    Args:
        side (ArmSide): Selects one of the two physical arms.
        body (ArmBrakeReleaseBody):

    Raises:
        errors.UnexpectedStatus: If the server returns an undocumented status code and Client.raise_on_unexpected_status is True.
        httpx.TimeoutException: If the request takes longer than Client.timeout.

    Returns:
        Response[ArmBrakeReleaseResponse200 | ErrorEnvelope]
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
    body: ArmBrakeReleaseBody,
) -> ArmBrakeReleaseResponse200 | ErrorEnvelope | None:
    """Release one arm's holding brakes for hand guiding (the arm DROPS)

     DANGEROUS. Forces this arm's holding brakes OPEN (the controller's `BRAK0`/`BRAK1` = 2 override, the
    vendor's recovery recipe for an arm twisted outside its limits) so a person can move it by hand. The
    servos are off, so **nothing holds the arm: it falls under gravity the instant the brakes open
    unless somebody is supporting it.**

    The body must carry `{"confirm": "RELEASE_BRAKE"}` (otherwise `400`). Accepted only while the arm's
    servos are off — mode `idle` or `error`; any other mode is a `409` naming the call that idles it.
    Gated by the arm soft-kill latch and, while somebody holds it, by the arm lease (`x-arm-lease`).
    REST-only.

    The release is TIMED: `seconds` (`1..=120`, default `30`) after it lands the daemon engages the
    brakes itself. Sending it again while released restarts the window. The brakes are also engaged by
    `/v1/arm/{side}/brake_engage`, by `/v1/arm/{side}/estop`, and by the soft kill (which the daemon
    also runs on shutdown). While released, `mode`, `recover` and a trajectory are refused on this arm,
    `ArmState` carries `brakes_released: true` and preflight lists it as blocking.

    Args:
        side (ArmSide): Selects one of the two physical arms.
        body (ArmBrakeReleaseBody):

    Raises:
        errors.UnexpectedStatus: If the server returns an undocumented status code and Client.raise_on_unexpected_status is True.
        httpx.TimeoutException: If the request takes longer than Client.timeout.

    Returns:
        ArmBrakeReleaseResponse200 | ErrorEnvelope
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
    body: ArmBrakeReleaseBody,
) -> Response[ArmBrakeReleaseResponse200 | ErrorEnvelope]:
    """Release one arm's holding brakes for hand guiding (the arm DROPS)

     DANGEROUS. Forces this arm's holding brakes OPEN (the controller's `BRAK0`/`BRAK1` = 2 override, the
    vendor's recovery recipe for an arm twisted outside its limits) so a person can move it by hand. The
    servos are off, so **nothing holds the arm: it falls under gravity the instant the brakes open
    unless somebody is supporting it.**

    The body must carry `{"confirm": "RELEASE_BRAKE"}` (otherwise `400`). Accepted only while the arm's
    servos are off — mode `idle` or `error`; any other mode is a `409` naming the call that idles it.
    Gated by the arm soft-kill latch and, while somebody holds it, by the arm lease (`x-arm-lease`).
    REST-only.

    The release is TIMED: `seconds` (`1..=120`, default `30`) after it lands the daemon engages the
    brakes itself. Sending it again while released restarts the window. The brakes are also engaged by
    `/v1/arm/{side}/brake_engage`, by `/v1/arm/{side}/estop`, and by the soft kill (which the daemon
    also runs on shutdown). While released, `mode`, `recover` and a trajectory are refused on this arm,
    `ArmState` carries `brakes_released: true` and preflight lists it as blocking.

    Args:
        side (ArmSide): Selects one of the two physical arms.
        body (ArmBrakeReleaseBody):

    Raises:
        errors.UnexpectedStatus: If the server returns an undocumented status code and Client.raise_on_unexpected_status is True.
        httpx.TimeoutException: If the request takes longer than Client.timeout.

    Returns:
        Response[ArmBrakeReleaseResponse200 | ErrorEnvelope]
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
    body: ArmBrakeReleaseBody,
) -> ArmBrakeReleaseResponse200 | ErrorEnvelope | None:
    """Release one arm's holding brakes for hand guiding (the arm DROPS)

     DANGEROUS. Forces this arm's holding brakes OPEN (the controller's `BRAK0`/`BRAK1` = 2 override, the
    vendor's recovery recipe for an arm twisted outside its limits) so a person can move it by hand. The
    servos are off, so **nothing holds the arm: it falls under gravity the instant the brakes open
    unless somebody is supporting it.**

    The body must carry `{"confirm": "RELEASE_BRAKE"}` (otherwise `400`). Accepted only while the arm's
    servos are off — mode `idle` or `error`; any other mode is a `409` naming the call that idles it.
    Gated by the arm soft-kill latch and, while somebody holds it, by the arm lease (`x-arm-lease`).
    REST-only.

    The release is TIMED: `seconds` (`1..=120`, default `30`) after it lands the daemon engages the
    brakes itself. Sending it again while released restarts the window. The brakes are also engaged by
    `/v1/arm/{side}/brake_engage`, by `/v1/arm/{side}/estop`, and by the soft kill (which the daemon
    also runs on shutdown). While released, `mode`, `recover` and a trajectory are refused on this arm,
    `ArmState` carries `brakes_released: true` and preflight lists it as blocking.

    Args:
        side (ArmSide): Selects one of the two physical arms.
        body (ArmBrakeReleaseBody):

    Raises:
        errors.UnexpectedStatus: If the server returns an undocumented status code and Client.raise_on_unexpected_status is True.
        httpx.TimeoutException: If the request takes longer than Client.timeout.

    Returns:
        ArmBrakeReleaseResponse200 | ErrorEnvelope
    """

    return (
        await asyncio_detailed(
            side=side,
            client=client,
            body=body,
        )
    ).parsed
